import builtins
import importlib

if hasattr(builtins, "SETTINGS"):
    module = importlib.import_module(builtins.SETTINGS)
    settings = { k: v for (k, v) in module.__dict__.items() if not k.startswith('_') }
    globals().update( settings )