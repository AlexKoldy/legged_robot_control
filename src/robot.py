import numpy as np
from pydrake.all import (
    StartMeshcat,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    AddMultibodyPlantSceneGraph,
    RigidTransform,
    DiagramBuilder,
    CoulombFriction,
    HalfSpace,
    Parser,
    Simulator,
)

from osc import OperationalSpaceController


class Robot:
    def __init__(self, filename: str, sim_time: int, remove_gravity: bool = False):
        """ """
        # Set up meshcat
        meshcat = StartMeshcat()

        # Set up the diagram builder. This wil handle the "block diagram"
        #  of our system and will allow us to connect the simulated and
        # experimental system
        builder = DiagramBuilder()
        self.plant, scene_graph = AddMultibodyPlantSceneGraph(
            builder, 0.001
        )  # simulated system

        # Set up ground
        self.plant.RegisterCollisionGeometry(
            self.plant.world_body(),
            RigidTransform(),
            HalfSpace(),
            "GroundCollisionGeometry",
            CoulombFriction(1.0, 1.0),
        )

        # Import the robot's model and finalize the plabt
        parser = Parser(self.plant)
        parser.AddModelFromFile(filename)

        import pydrake

        print(type(parser))
        print(pydrake.__version__)
        exit()

        # Remove gravity for testing
        if remove_gravity:
            plant_gravity_field = self.plant.gravity_field()
            plant_gravity_field.set_gravity_vector([0, 0, 0])

        # Finalize plant
        self.plant.Finalize()

        # Add planner and controller to builder
        osc = builder.AddSystem(OperationalSpaceController(filename))
        builder.Connect(self.plant.get_state_output_port(), osc.get_state_input_port())
        builder.Connect(osc.get_output_port(), self.plant.get_actuation_input_port())

        # Set up some visualizations stuff
        vis_params = MeshcatVisualizerParams(publish_period=0.01)
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, params=vis_params)
        self.diagram = builder.Build()

        # Simulate the system
        self.simulate(sim_time)

    def simulate(self, sim_time: int):
        """
        Runs the Drake simulation for the system

        Arguments:
            sim_time (int) - total simulation time
        """
        simulator = Simulator(self.diagram)
        simulator.Initialize()
        simulator.set_target_realtime_rate(1)

        # Set initial pose
        context = self.diagram.GetMutableSubsystemContext(
            self.plant, simulator.get_mutable_context()
        )
        # TODO: remove

        # M = self.plant.CalcMassMatrix(context)
        # print(M.shape)
        # print(self.plant.num_velocities())
        # B = self.plant.MakeActuationMatrix()
        # print(B.shape)
        # print(self.plant.num_actuators())

        q = self.plant.GetPositions(context)
        q[6] = 0.93845
        self.plant.SetPositions(context, q)

        # TODO: remove
        # com_rf = self.plant.GetBodyByName("r_foot").CalcCenterOfMassInBodyFrame(context)
        # print(com_rf)

        simulator.AdvanceTo(sim_time)


if __name__ == "__main__":
    filename = "urdf/atlas_minimal_contact.urdf"
    sim_time = 1000  # [s]
    atlas = Robot(filename, sim_time)
