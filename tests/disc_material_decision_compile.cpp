#include "disc_material_decision.h"

constexpr int round1[] = {1, 5, 6};
constexpr int round2[] = {5, 1, 6};
static_assert(ShouldGrabTaskColor(round1, 0, 1, true), "first color matches");
static_assert(!ShouldGrabTaskColor(round1, 0, 5, true), "later task color must wait");
static_assert(ShouldGrabTaskColor(round1, 1, 5, true), "next color after progress");
static_assert(!ShouldGrabTaskColor(round1, 1, 1, true), "already completed color must skip");
static_assert(ShouldGrabTaskColor(round1, 2, 6, true), "third color matches");
static_assert(ShouldGrabTaskColor(round2, 0, 5, true), "second round uses its own sequence");
static_assert(!ShouldGrabTaskColor(round2, 0, 1, true), "first round sequence not reused");
static_assert(!ShouldGrabTaskColor(round1, 3, 1, true), "finished round skips");
static_assert(!ShouldGrabTaskColor(round1, -1, 1, true), "invalid progress skips");
static_assert(!ShouldGrabTaskColor(round1, 0, 1, false), "invalid task skips");
static_assert(!ShouldGrabTaskColor(nullptr, 0, 1, true), "no round selected skips");
static_assert(!ShouldGrabTaskColor(round1, 0, 0, true), "invalid color skips");

constexpr int manualRound[] = {2, 3, 5};
static_assert(AdvanceDiscGrabProgress(0, 1, true) == 1, "first grab advances once");
static_assert(AdvanceDiscGrabProgress(1, 2, false) == 1, "failure retains next color");
static_assert(AdvanceDiscGrabProgress(1, 1, true) == 1, "old cargo cannot advance twice");
static_assert(AdvanceDiscGrabProgress(2, 3, true) == 3, "third successful grab completes round");
static_assert(AdvanceDiscGrabProgress(3, 3, true) == 3, "completed round cannot advance");
static_assert(EvaluateDiscColor(manualRound, AdvanceDiscGrabProgress(0, 1, true), 3, true, false, true, false) == DiscColorDecision::GRAB, "color 3 follows completed color 2");
static_assert(EvaluateDiscColor(manualRound, 0, 2, true, false, true, false) == DiscColorDecision::GRAB, "235 task grabs color 2 first");
static_assert(EvaluateDiscColor(manualRound, 0, 2, true, false, true, true) == DiscColorDecision::OCCUPIED, "matching color blocked by occupied cargo");
static_assert(EvaluateDiscColor(manualRound, 0, 2, true, true, true, false) == DiscColorDecision::ALIGN_ACTIVE, "matching color blocked by alignment");
static_assert(EvaluateDiscColor(manualRound, 0, 3, true, true, true, true) == DiscColorDecision::COLOR_MISMATCH, "only wrong task color skips");
static_assert(EvaluateDiscColor(manualRound, 0, 2, false, false, true, false) == DiscColorDecision::NO_TASK, "missing task is explicit");
static_assert(EvaluateDiscColor(nullptr, 0, 2, true, false, true, false) == DiscColorDecision::NOT_WAITING, "wrong phase is explicit");
static_assert(EvaluateDiscColor(manualRound, 3, 2, true, false, true, false) == DiscColorDecision::ROUND_COMPLETE, "completed round is explicit");
static_assert(EvaluateDiscColor(manualRound, 0, 2, true, false, false, false) == DiscColorDecision::NOT_READY, "unprepared first round is explicit");
