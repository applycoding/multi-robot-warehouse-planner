from hypothesis import settings

settings.register_profile(
    "planner",
    max_examples=100,
    deadline=None,
    derandomize=True,
    database=None,
)
settings.register_profile(
    "simulation",
    max_examples=30,
    deadline=None,
    derandomize=True,
    database=None,
)
