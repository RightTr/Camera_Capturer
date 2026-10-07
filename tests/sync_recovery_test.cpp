// Real synchronization paths, without opening devices or a ROS graph.
#define main rgbdt_node_main
#include "../src/ros/RGBDT_trigger_ros.cpp"
#undef main
#include <cassert>

int main()
{
    using namespace std::chrono_literals;
    const auto now = SteadyClock::now();
    SyncRecovery timer;
    timer.restart(now);
    assert(!timer.warning_due(now + 9s));
    timer.restart(now + 9s);
    assert(timer.warning_due(now + 10s));
    assert(!timer.warning_due(now + 14s));
    assert(timer.warning_due(now + 15s));

    restart_preroll("test startup");
    trigger_period_ns = 10000000;
    rgbd_slot = {true, 40, 1087};
    const auto old_epoch = epoch;
    StampedRealSenseFrame gap;
    gap.color_frame_number = 1086;
    gap.depth_frame_number = 1089;
    gap.trigger_step = 2; // Incident: color +1, depth +2.
    auto arrival = epoch_start_host_ns + 2 * trigger_period_ns;
    gap.color_host_sec = arrival / 1000000000;
    gap.color_host_nanosec = arrival % 1000000000;
    imu_frames.emplace_back();
    assert(accept_rgbd_locked(gap));
    assert(epoch == old_epoch && imu_frames.size() == 1);
    triggers.push_back({40, 1000000000, arrival});
    triggers.push_back({41, 1010000000, arrival + trigger_period_ns});
    auto assigned = assign_slot(1089, arrival, rgbd_slot, epoch);
    assert(assigned && assigned->id == 41 && rgbd_slot.last_sequence == 1089);
    // No renumbering, restart, or clearing of continuous IMU after the skip.
    assert(epoch == old_epoch && imu_frames.size() == 1 && !fatalFlag);

    const auto seed_imu = [](std::uint64_t base, std::uint64_t duration) {
        imu_frames.clear();
        imu_stream_fps = {200, 200};
        for (std::uint64_t t = base; t <= base + duration; t += 5000000) {
            imu_frames.push_back({RS2_STREAM_ACCEL, 0, t, 0, 0, 9.8f, t, 200});
            imu_frames.push_back({RS2_STREAM_GYRO, 0, t, 0, 0, 1, t, 200});
        }
        for (auto& track : imu_tracks)
            track = {true, 1, base + duration, SteadyClock::now()};
    };
    // Removing a depth IMAGE anchor must not remove any physical IMU sample.
    seed_imu(1000000000, 40000000);
    std::deque<MappedImu> mapped;
    std::deque<WriteJob> pending;
    std::int64_t last = 0;
    if_save = 1;
    assert(map_interval_locked({1,1000000000,2000000000},
                               {2,1010000000,2010000000},mapped,last,pending));
    assert(map_interval_locked({2,1010000000,2010000000},
                               {4,1030000000,2030000000},mapped,last,pending));
    assert(mapped.size() == 6 && pending.size() == 12 && writer_jobs.empty());
    double angle = 0;
    for (std::size_t i = 0; i < mapped.size(); ++i) {
        assert(mapped[i].stamp_ns == 2000000000 + static_cast<std::int64_t>(i)*5000000);
        if (i) angle += mapped[i].message.angular_velocity.z *
            (mapped[i].stamp_ns - mapped[i-1].stamp_ns) * 1e-9;
    }
    assert(std::abs(angle - .025) < 1e-12); // Identical to a complete 200 Hz stream.
    assert(!fatalFlag && epoch == old_epoch);
    if_save = 0;
    restart_preroll("coordinator test");
    g_output_started = true;
    seed_imu(3000000000, 50000000);
    // Depth image at n=1 is absent; Guide n=1 must still publish.
    for (std::uint64_t n=0; n<4; ++n) {
        CameraSlot slot;
        slot.trigger = {n,4000000000+static_cast<std::int64_t>(n*10000000),0};
        slot.left.emplace(); slot.right.emplace();
        if (n!=1) {
            slot.rgbd.emplace();
            slot.rgbd->depth_sensor_ns=3000000000+n*10000000;
            depth_anchors.push_back(slot.anchor());
        }
        slots.emplace(n,std::move(slot));
    }
    const auto started_epoch = epoch;
    std::thread coordinator(coordinator_loop);
    {
        std::unique_lock<std::mutex> lock(sync_mutex);
        assert(sync_cv.wait_for(lock,2s,[] { return published_imu_count>=6 || quitFlag.load(); }));
        assert(!quitFlag && published_imu_count==6 && published_guide_pairs==2);
        assert(published_rgbd_pairs==1 && max_published_imu_dt_ns==5000000);
        assert(epoch==started_epoch);
        // A prolonged thermal outage must not gate the IMU clock/output.
        depth_anchors.push_back({4,3040000000,4040000000});
        sync_cv.notify_all();
        assert(sync_cv.wait_for(lock,2s,[] { return published_imu_count>=8 || quitFlag.load(); }));
        assert(!quitFlag && published_imu_count==8 && max_published_imu_dt_ns==5000000);
        // A real broken IMU/timebase cannot silently resume into Air-VINS.
        reset_preroll_locked("Injected physical IMU gap");
        assert(quitFlag && fatalFlag && epoch==started_epoch);
    }
    coordinator.join();
    // Recording failures still stop explicitly, without silently losing samples.
    quitFlag=false; fatalFlag=false; if_save=1; writer_capacity=0;
    assert(!enqueue_write(WriteJob{WriteJob::Kind::imu}));
    assert(quitFlag && fatalFlag);
}
