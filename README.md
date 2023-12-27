# cf_motors

## Setup development environment

### pre-commit

#### Install pre-commit

```bash
pip3 install pre-commit
```

or (for MacOS)

```bash
brew install pre-commit
```

#### Install pre-commit hooks

```bash
pre-commit install
```
More information about pre-commit hooks can be found [here](https://pre-commit.com/).

### Build

```bash
mkdir build && cd build
cmake ..
make
```
