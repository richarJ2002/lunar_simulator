"""!
@brief  Marks this directory as a test package. `unittest discover
        -s test/post_processing` (this project's documented verification
        command) imports test modules as bare top-level names rather than
        through this package, so each test module makes
        `post_processing/python_tools` importable for itself at its own
        top -- the same `sys.path` adjustment every post_processing_*.py
        entry point already makes for itself -- rather than relying on
        this file to do it once.
"""
