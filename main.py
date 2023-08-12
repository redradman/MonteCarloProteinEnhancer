import os
import pyrosetta
from pyrosetta import Pose
from pyrosetta.rosetta.core.scoring import ScoreFunctionFactory
from pyrosetta.rosetta.core.pack.task import TaskFactory
from pyrosetta.rosetta.core.pack.task.operation import IncludeCurrent, RestrictToRepacking
from pyrosetta.rosetta.protocols.minimization_packing import PackRotamersMover, MinMover, RotamerTrialsMover
from pyrosetta.rosetta.core.kinematics import MoveMap
from pyrosetta.rosetta.protocols.moves import MonteCarlo
from pyrosetta.rosetta.protocols.simple_moves import SmallMover, ShearMover

# Initialize PyRosetta
pyrosetta.init()

# Load the protein pose
pose = Pose()
pyrosetta.rosetta.core.import_pose.pose_from_file(pose, "pdb_files/sample/1ubq.pdb")
original_pose = pose.clone()

# Score Function
score_function = ScoreFunctionFactory.create_score_function("ref2015.wts")

# Set up backbone movers: SmallMover and ShearMover
small_mover = SmallMover()
shear_mover = ShearMover()

# Set up sidechain mover: RotamerTrialsMover
task_factory = TaskFactory()
task_factory.push_back(IncludeCurrent())
task_factory.push_back(RestrictToRepacking())
packer_task = task_factory.create_task_and_apply_taskoperations(pose)
rotamer_trials = RotamerTrialsMover(score_function, packer_task)

# PackRotamersMover for repacking
packer = PackRotamersMover(score_function, packer_task)

# Set up MinMover for energy minimization
movemap = MoveMap()
movemap.set_bb(True)
movemap.set_chi(True)
min_mover = MinMover()
min_mover.score_function(score_function)
min_mover.movemap(movemap)

# Set up MonteCarlo object with the desired temperature
mc = MonteCarlo(pose, score_function, 1.0)

# Define output directory
output_dir = "output_decoys"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)