from setuptools import setup, find_packages

with open('README.md', 'r') as f:
    description = f.read()

setup(
    name='mlops_api_gemstack',
    version='0.4.1',
    description='An API package connect to a MLops server relating to the GEMstack project.',
    author='Haoming Sun',
    author_email='ricardosun990122@gmail.com',
    url='https://github.com/krishauser/GEMstack/tree/s2024_MLops/MLops/mlops_api_gemstack',
    packages=find_packages(),
    install_requires=[
        'requests',
        'tqdm',
        'pprintpp',
        'urllib3',
        'click'
    ],
    entry_points={
        'console_scripts': [
            'mlops=mlops_api_gemstack:cli'
        ]
    },
    long_description=description,
    long_description_content_type='text/markdown',
)