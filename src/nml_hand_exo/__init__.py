"""
nml_hand_exo: Python package for controlling the NML HandExo device.

"""

__version__ = "0.0.5"
__author__ = "Neuromechatronics Lab"
__email__ = "neuromech@andrew.cmu.edu"
__license__ = "MIT"
__url__ = "https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo"
__description__ = "Python package for controlling the NML HandExo device."

submodules = [
    'applications',
    'interface',
    'processing',
    'plotting',

]

__all__ = submodules + [
    '__version__',
]

def __dir__():
    return __all__

