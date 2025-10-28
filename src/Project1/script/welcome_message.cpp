#include <gazebo/gazebo.hh> // Include the main Gazebo header file

namespace gazebo
{	
	// Define a new class WorldPluginMyRobot tat inherits from the WorldPlugin class
	class WorldPluginMyRobot : public WorldPlugin
	{
	public:
		// Constructor for WorldPluginMyRobot
		WorldPluginMyRobot() : WorldPlugin()
		{
			printf("Welcome to Hafizur Rahman's world!'\n");
		}
	public:
		// Load function is called by Gazebo when loading the plugin
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
		{
		}
	};
	// Register this plugin with Gazebo, 
	// making it discoverable at runtime
	GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)
}
