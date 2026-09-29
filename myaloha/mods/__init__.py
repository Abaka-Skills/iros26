from myaloha.mods.base import Mod
from myaloha.mods.instruction import Instruction
from myaloha.mods.iros26 import Iros26
from myaloha.mods.localize import Localize
from myaloha.mods.pick_place import PickPlace
from myaloha.mods.reset import Reset
from myaloha.mods.teleop import Teleop

MODS = {m.name: m for m in (Reset, Teleop, Instruction, Localize, PickPlace, Iros26)}
