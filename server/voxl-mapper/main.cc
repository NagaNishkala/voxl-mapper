#include "voxl_mapper.h"
#include "config_file.h"
#include <unistd.h>
#include <getopt.h>
#include <sys/stat.h>	// for mkdir
#include <sys/types.h>	// for mode_t in mkdir
#include <fstream>
#include <iostream>


bool en_debug= false;
bool en_timing= false;

inline bool exists_ (const char *name)
{
    std::ifstream f(name);
    return f.good();
}

static int _mkdir(const char *dir)
{
	char tmp[PATH_MAX];
	char *p = NULL;

	snprintf(tmp, sizeof(tmp), "%s", dir);
	for (p = tmp + 1; *p != 0; p++)
	{
		if (*p == '/')
		{
			*p = 0;
			if (mkdir(tmp, S_IRWXU) && errno != EEXIST)
			{
				perror("ERROR calling mkdir");
				printf("tried to mkdir %s\n", tmp);
				return -1;
			}
			*p = '/';
		}
	}
	return 0;
}

static void _print_usage(void){
    	printf("\n\
voxl-mapper usually runs as a systemd background service. However, for debug\n\
purposes it can be started from the command line manually with any of the following\n\
debug options. When started from the command line, voxl-mapper will automatically\n\
stop the background service so you don't have to stop it manually\n\
\n\
-c, --config          load the config file only, for use by the config wizard\n\
-d, --debug           run in debug mode.\n\
-t, --timing          runs in timing mode. Prints how long each step takes.\n\
-h, --help            print this help message\n\
\n");
	return;
}

static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
        {"config",				    no_argument,		    0, 'c'},
        {"debug",					no_argument,		    0, 'd'},
        {"timing",					no_argument,		    0, 't'},
        {"help",					no_argument,		    0, 'h'},
	};

	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "cm:i:s:dthz", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;

		case 'c':
			config_file_read();
			exit(0);
			break;

        case 'd':
            en_debug = true;
			break;

        case 't':
            en_timing = true;
			break;

		case 'h':
			_print_usage();
			return -1;

		default:
			_print_usage();
			return -1;
		}
	}
	return 0;
}

int main(int argc, char** argv) {
    if(_parse_opts(argc, argv)) return -1;

	printf("Loading our own config file\n");
    if(config_file_read()) return -1;
    if (en_debug) config_file_print();

    printf("Loading extrinsics config file\n");
	if(load_extrinsics_file(en_debug, depth_modes(depth_mode))) return -1;

    _mkdir("/data/voxl_mapper/");

    if (!exists_(esdf_save_path)){
        std::ofstream outfile (esdf_save_path);
        outfile.close();
    }
    if (!exists_(tsdf_save_path)){
        std::ofstream outfile (tsdf_save_path);
        outfile.close();
    }
    if (!exists_(mesh_save_path)){
        std::ofstream outfile (mesh_save_path);
        outfile.close();
    }
    // setup tsdf configs
    voxblox::TsdfMap::Config map_config;
    voxblox::TsdfIntegratorBase::Config int_config;
    map_config.tsdf_voxel_size = voxel_size;
    map_config.tsdf_voxels_per_side = voxels_per_side;

    if(enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

    voxblox::MeshIntegratorConfig mesh_config;

    printf("Trying to init tsdf server\n");
    // initialize tsdf server
    voxblox::TsdfServer tsdf_server(map_config, int_config, mesh_config, en_debug, en_timing, depth_modes(depth_mode));
    printf("created tsdf server\n");
	// indicate to the soon-to-be-started thread that we are initialized
	// and running, this is an extern variable in start_stop.c
    main_running = 1;

    //start recieving and integrating data
    tsdf_server.initMPA();

	////////////////////////////////////////////////////////////////////////////////
	// main loop just waits and keeps trying to open pipes if they are closed
	////////////////////////////////////////////////////////////////////////////////
    while(main_running){
        usleep(500000);
    }
    ////////////////////////////////////////////////////////////////////////////////
	// close everything
	////////////////////////////////////////////////////////////////////////////////
    tsdf_server.closeMPA();
    return 0;
}
