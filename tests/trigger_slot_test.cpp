#include <cassert>
#include <deque>

#include "utils/trigger_slot.h"

int main()
{
    const std::deque<Trigger> triggers{{100, 1000000000, 1000000000},
                                       {101, 1033333333, 1033333333},
                                       {102, 1066666666, 1066666666}};
    // The future edge closes the arrival interval; host latency only seeds
    // the initial slot and never determines subsequent slots.
    assert(calibration_trigger(triggers, 1010000000, 25000000)->id == 100);
    assert(calibration_trigger(triggers, 1040000000, 25000000)->id == 101);
    assert(calibration_trigger(triggers, 1060000000, 25000000) == nullptr);
    assert(calibration_trigger(triggers, 1029000000, 25000000) == nullptr);
    assert(calibration_trigger({triggers[0]}, 1010000000, 25000000) == nullptr);
    assert(calibration_trigger(triggers, 999000000, 25000000) == nullptr);

    StreamSlot thermal{true, 100, 10};
    Trigger assigned;
    assert(advance_slot(triggers, 11, thermal, assigned) == SlotAdvance::ready);
    assert(assigned.id == 100 && thermal.next_id == 101);
    assert(advance_slot(triggers, 13, thermal, assigned) == SlotAdvance::sequence_gap);
    assert(thermal.next_id == 101);
    assert(advance_slot(triggers, 12, thermal, assigned) == SlotAdvance::ready);
    assert(assigned.id == 101);
    assert(advance_slot({triggers[2]}, 13, thermal, assigned) == SlotAdvance::ready);

    StreamSlot lagged{true, 100, 10};
    assert(advance_slot({triggers[1], triggers[2]}, 11, lagged, assigned) ==
           SlotAdvance::expired);
    assert(advance_slot({triggers[0]}, 11, lagged, assigned) == SlotAdvance::ready);
    assert(advance_slot({triggers[0]}, 12, lagged, assigned) == SlotAdvance::waiting);
}
