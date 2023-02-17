# Robot workcell description package

## Contributing
This repository uses pre-commit for code formatting. This program has to be setup locally and installed inside the repository. For this execute in the repository folder following commands:
```bash
sudo apt install -y pre-commit
pre-commit install
```
The check are automatically executed before each commit. This helps you to always commit well formatted code. To run all the checks manually use ``pre-commit run -a`` command. For the other options check ``pre-commit --help``.

In a case of an "emergency" you can avoid execution of pre-commit hooks by adding ``-n`` flag to git commit command - this is NOT recommended to do if you don't know what are you doing!
