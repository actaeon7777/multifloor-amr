"""PPO for the AMR navigation task: Isaac Lab rsl_rl defaults, MLP actor/critic, bounded action noise."""
from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class BoundedGaussianCfg(RslRlMLPModelCfg.GaussianDistributionCfg):
    """Gaussian with clamped std. rsl_rl's GaussianDistribution accepts std_range; Isaac Lab's cfg omits it.

    The env clips actions to [-1, 1], so samples beyond it cost nothing while the entropy bonus
    rewards a wider std: in the first v1 run the std grew 0.3 -> 4.6 (v) / 1.8 (w) and nearly
    every training action was saturated. Capping it below the action range prevents that.
    """

    std_range: tuple[float, float] = (0.05, 0.8)


@configclass
class NavArenaPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 1500
    save_interval = 100
    experiment_name = "amr_nav_arena"
    obs_groups = {"actor": ["policy"], "critic": ["policy"]}
    clip_actions = 1.0
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=BoundedGaussianCfg(init_std=0.6),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[256, 128], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.001,  # 0.005 kept pushing the std up (see BoundedGaussianCfg)
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
