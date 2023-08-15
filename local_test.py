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

print(os.getcwd())

# Load the protein pose
pose = Pose()
pyrosetta.rosetta.core.import_pose.pose_from_file(pose, "pdb_files/dockedNrdj-1mCherry.pdb")
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

# Placeholder variables to keep track of the best score and its iteration
best_score = float('inf')
best_iteration = -1

# Simulation
for i in range(10):
    pose = original_pose.clone()

    # Inner loop for perturbations and optimizations
    for _ in range(10):
        small_mover.apply(pose)
        shear_mover.apply(pose)
        rotamer_trials.apply(pose)
        packer.apply(pose)
        min_mover.apply(pose)
        accepted = mc.boltzmann(pose)
        
        # Update the best score and iteration if needed
        current_score = score_function(pose)
        if current_score < best_score:
            best_score = current_score
            best_iteration = i

    # Save all decoys
        pose.dump_pdb(os.path.join(output_dir, f"decoy_{i+1}.pdb"))

# Save the best-scoring pose
best_pose = mc.lowest_score_pose()
best_pose.dump_pdb(os.path.join(output_dir, f"best_decoy_at_iteration_{best_iteration}.pdb"))
