from setuptools import Extension, setup

setup(
    python_requires=">=3.6",
    install_requires=['gym==0.11', 'ruamel.yaml==0.16.10',
                      'numpy', 'stable_baselines==2.10.1'],
    packages=['rl.lxx_baselines.ppo', 'rl.lxx_baselines.common', 'rl.lxx_baselines.envs']
)
