from GEMstack.utils import settings,config
import sys

if __name__=='__main__':
    #check for settings override
    for arg in sys.argv[1:]:
        if arg.startswith('--settings='):
            settings.load_settings(arg[11:])
            break
    #get launch file
    launch_file = None
    for arg in sys.argv[1:]:
        if arg.startswith('--run='):
            launch_file = arg[6:]
            break
        elif not arg.startswith('--'):
            launch_file = arg
            break
    if launch_file is None:
        runconfig = settings.get('run',None)
        if runconfig is None:
            print("Usage: python3 [--key1=value1 --key2=value2] [--settings=SETTINGS_OVERRIDE.yaml] LAUNCH_FILE.yaml")
            print("  Default settings are found in knowledge/defaults/current.yaml")
            exit(1)
        else:
            print("Using default run configuration in knowledge/defaults/current.yaml")
    else:
        #set the run settings from command line
        run_config = config.load_config_recursive(launch_file)
        settings.set('run',run_config)
        if settings.get('run.name',None) is None:
            print("yes")
            settings.set('run.name',launch_file)

    from GEMstack.onboard.execution import entrypoint
    entrypoint.main()
