#include "task_code.h"

static_assert(IsTaskCodeValid("156+123+516+231"), "valid task");
static_assert(IsTaskCodeValid("111+321+666+132"), "valid color bounds and position permutations");
static_assert(!IsTaskCodeValid(nullptr), "null rejected");
static_assert(!IsTaskCodeValid(""), "empty rejected");
static_assert(!IsTaskCodeValid("156+123+516+231x"), "trailing garbage rejected");
static_assert(!IsTaskCodeValid("156+123+516+23"), "short task rejected");
static_assert(!IsTaskCodeValid("156-123+516+231"), "separator rejected");
static_assert(!IsTaskCodeValid("056+123+516+231"), "color zero rejected");
static_assert(!IsTaskCodeValid("156+123+716+231"), "color seven rejected");
static_assert(!IsTaskCodeValid("156+112+516+231"), "duplicate positions rejected");
static_assert(!IsTaskCodeValid("156+123+516+241"), "position four rejected");
static_assert(!IsTaskCodeValid("a56+123+516+231"), "non digit rejected");
