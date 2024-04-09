from setuptools import setup, find_packages

setup(
    name='gemstack_api_package',
    version='0.0.1',
    description='A Python client for the API to a Gemstack server',
    author='Haoming Sun',
    author_email='ricardosun990122@gmail.com',
    url='https://github.com/krishauser/GEMstack/tree/s2024_MLops/MLops/api_package',
    packages=find_packages(),
    install_requires=[
        'requests',
        'tqdm',
        'pprint',
        'urllib'
    ],
)