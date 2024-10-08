from setuptools import Extension, setup

setup(
    name ="SnowyOwl",
    version = "3.0.0",
    python_requires=">=3.6",
    install_requires=['gym==0.11', 'ruamel.yaml==0.16.10',
                      'numpy==1.22.0', 'stable_baselines==2.10.1'],
    packages=['rl.lxx_baselines.ppo', 'rl.lxx_baselines.common', 'rl.lxx_baselines.envs']
)
