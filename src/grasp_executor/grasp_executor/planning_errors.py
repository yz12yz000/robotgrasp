"""Only explicit, completed planning rejections permit another contact."""


class CandidatePlanRejected(RuntimeError):
    def __init__(self, reason, *, stage="planning", reasons=None):
        super().__init__(reason)
        self.stage = stage
        self.reasons = tuple(reasons or (reason,))

