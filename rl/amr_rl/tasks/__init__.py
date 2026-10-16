"""AMR navigation tasks, loaded by `isaaclab train` through the `isaaclab.tasks` entry point."""
import gymnasium as gym

gym.register(
    id="AMR-Nav-Arena-v0",
    entry_point=f"{__name__}.nav_arena_env:NavArenaEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.nav_arena_env_cfg:NavArenaEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:NavArenaPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
