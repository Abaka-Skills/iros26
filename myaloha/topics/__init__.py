from myaloha.topics.cam_params import CamParams
from myaloha.topics.locate_cubes import LocateCubes

TOPICS = {t.name: t for t in (CamParams, LocateCubes)}
