from myaloha.mods.base import Mod
from myaloha.mods.instruction import Instruction
from myaloha.mods.reset import Reset
from myaloha.mods.teleop import Teleop

MODS = {m.name: m for m in (Reset, Teleop, Instruction)}
