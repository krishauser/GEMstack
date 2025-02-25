from GEMstack.utils import settings,config
import sys

if __name__=='__main__':
    launch_file = None
    for arg in sys.argv[1:]:
        if arg.startswith('--run='):
            launch_file = arg[9:]
            break
        elif not arg.startswith('--'):
            launch_file = arg
            break
    if launch_file is None:
        runconfig = settings.get('run',None)
        if runconfig is None:
            print("Usage: python3 [--key1=value1 --key2=value2] LAUNCH_FILE.yaml")
            print("  Current settings are found in knowledge/defaults/current.yaml")
            exit(1)
        else:
            print("Using default run configuration in knowledge/defaults/current.yaml")
    else:
        #set the run settings from command line
        run_config = config.load_config_recursive(launch_file)
        #print(run_config)
        settings.set('run',run_config)
        if settings.get('run.name',None) is None:
            settings.set('run.name',launch_file)

    from GEMstack.onboard.execution import entrypoint
    entrypoint.main()
