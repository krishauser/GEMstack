# OneDrive Upload

## Config file setup

An authenticator service must be setup in azure first. This can be done by going to App registrations in Azure and creating a new resgistration


To configure the onedrive config, it must be named onedrive_config.json and be in the following format

```json
{
  "CLIENT_ID": "",
  "TENANT_ID": "",
  "DRIVE_ID": "",
  "ITEM_ID": ""
}
```

The Client Id and the tenant id of the authenticator service found in the app registration in azure

The DRIVE_ID and ITEM_ID of the onedrive folder to upload to can be found with this guide:
https://www.youtube.com/watch?v=3pjS7YTIcgQ

Anybody can access the same authenticator service for free as long as they have an illinois account.


## Usage
When any Ros topics are specified in the yaml file under 'log,' a prompt will appear in the console after finishing the GEMStack execution. If the user specifies that they want to continue upload, a window will appear in browser (will not work in docker or when running headless.) You may login with your illinois account only. 
