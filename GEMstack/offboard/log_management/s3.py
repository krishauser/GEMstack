#!/usr/bin/env python3
"""
This client interacts with S3 to upload (push) or download (pull) a specific folder.

Before running this client make sure you have defined .env in root with following values:
AWS_ACCESS_KEY_ID=example
AWS_SECRET_ACCESS_KEY=example
AWS_DEFAULT_REGION=us-east-1

Requires:
    pip install boto3 python-dotenv

Example Usage:

  # Push a folder named "2025-02-12_15-30-00" inside "data/" to S3:
  python3 GEMStack/offboard/log_management/s3.py \
    --action push \
    --folder 2025-02-12_15-30-00 \
    --bucket cs588 \
    --s3-prefix captures

  # Pull (download) that same folder from S3 into local "download/" directory:
  python3 GEMStack/offboard/log_management/s3.py \
    --action pull \
    --folder 2025-02-12_15-30-00 \
    --bucket cs588 \
    --s3-prefix captures \
    --dest-dir download
"""

import argparse
import boto3
import os
import sys
from dotenv import load_dotenv

def get_s3_client():
    """
    Initializes the S3 client using AWS credentials from environment variables.
    Expects:
      - AWS_ACCESS_KEY_ID
      - AWS_SECRET_ACCESS_KEY
      - AWS_DEFAULT_REGION
    Exits if any of these are missing.
    """
    # load environment variables from .env file
    # override in case local config exists for AWS
    load_dotenv(override=True)

    access_key = os.environ.get('AWS_ACCESS_KEY_ID')
    secret_key = os.environ.get('AWS_SECRET_ACCESS_KEY')
    region = os.environ.get('AWS_DEFAULT_REGION')

    if not access_key or not secret_key or not region:
        sys.exit(
            "Error: AWS credentials not set. Please set AWS_ACCESS_KEY_ID, "
            "AWS_SECRET_ACCESS_KEY, and AWS_DEFAULT_REGION environment variables (in .env)."
        )

    print(access_key, secret_key, region)
    return boto3.client(
        's3',
        aws_access_key_id=access_key,
        aws_secret_access_key=secret_key,
        region_name=region
    )

def check_s3_connection(s3_client, bucket):
    """
    Verifies that we can connect to S3 and access the specified bucket.
    """
    try:
        s3_client.head_bucket(Bucket=bucket)
        print(f"Connection check: Successfully accessed bucket '{bucket}'")
    except Exception as e:
        sys.exit(f"Error: Could not connect to S3 bucket '{bucket}': {e}")

def push_folder_to_s3(folder_path, bucket, s3_prefix):
    """
    Walks through the folder and uploads each file to S3 under the given prefix.
    Files are stored under the key: s3_prefix/folder_name/<relative_file_path>.
    """
    s3_client = get_s3_client()
    check_s3_connection(s3_client, bucket)

    folder_name = os.path.basename(folder_path)
    files_uploaded = 0

    for root, _, files in os.walk(folder_path):
        for file in files:
            local_path = os.path.join(root, file)
            # determine the file's path relative to the folder being pushed.
            relative_path = os.path.relpath(local_path, folder_path)
            s3_key = os.path.join(s3_prefix, folder_name, relative_path)
            try:
                print(f"Uploading {local_path} to s3://{bucket}/{s3_key}")
                s3_client.upload_file(local_path, bucket, s3_key)
                files_uploaded += 1
            except Exception as e:
                print(f"Error uploading {local_path}: {e}")

    if files_uploaded == 0:
        sys.exit(f"Error: No files were uploaded from folder: {folder_path}")

def pull_folder_from_s3(bucket, s3_prefix, folder_name, dest_dir):
    """
    Downloads all objects from S3 whose key begins with s3_prefix/folder_name.
    The folder will be recreated under `dest_dir/folder_name`.
    """
    s3_client = get_s3_client()
    check_s3_connection(s3_client, bucket)

    # this prefix is what we'll look for in the bucket.
    prefix = os.path.join(s3_prefix, folder_name)

    paginator = s3_client.get_paginator('list_objects_v2')
    pages = paginator.paginate(Bucket=bucket, Prefix=prefix)

    found_files = False
    for page in pages:
        if 'Contents' not in page:
            continue
        for obj in page['Contents']:
            key = obj['Key']
            # if the object key is exactly the "folder" (a directory placeholder), skip it.
            # e.g. sometimes you can have an empty key or just a trailing slash.
            if key.endswith("/"):
                continue

            found_files = True
            # figure out the relative path to recreate the same structure locally.
            relative_path = os.path.relpath(key, prefix)
            local_path = os.path.join(dest_dir, folder_name, relative_path)

            # ensure the local directory exists.
            os.makedirs(os.path.dirname(local_path), exist_ok=True)

            print(f"Downloading s3://{bucket}/{key} to {local_path}")
            try:
                s3_client.download_file(bucket, key, local_path)
            except Exception as e:
                print(f"Error downloading s3://{bucket}/{key}: {e}")

    if not found_files:
        sys.exit(f"Error: No files found in bucket '{bucket}' with prefix '{prefix}'")

def main():
    parser = argparse.ArgumentParser(
        description="Push or pull a specific folder to/from an S3 bucket."
    )
    parser.add_argument("--action", choices=["push", "pull"], required=True,
                        help="Choose whether to push (upload) or pull (download) a folder.")
    parser.add_argument("--folder", required=True,
                        help="Folder name (e.g. 2025-02-12_15-30-00)")
    parser.add_argument("--base-dir", default="data",
                        help="Local base directory where capture runs are stored (for push).")
    parser.add_argument("--dest-dir", default="download",
                        help="Local directory to place the downloaded folder (for pull).")
    parser.add_argument("--bucket", required=True,
                        help="S3 bucket name.")
    parser.add_argument("--s3-prefix", default="captures",
                        help="S3 prefix (folder) where data is stored or will be uploaded.")
    args = parser.parse_args()

    if args.action == "push":
        # pushing from local 'base_dir/folder' to S3
        folder_path = os.path.join(args.base_dir, args.folder)
        if not os.path.exists(folder_path):
            sys.exit(f"Error: Folder does not exist: {folder_path}")

        # check if the folder is empty.
        folder_empty = True
        for _, _, files in os.walk(folder_path):
            if files:
                folder_empty = False
                break
        if folder_empty:
            sys.exit(f"Error: Folder is empty: {folder_path}")

        push_folder_to_s3(folder_path, args.bucket, args.s3_prefix)

    elif args.action == "pull":
        # pulling from S3 (prefix/folder) to local 'dest_dir/folder'
        pull_folder_from_s3(args.bucket, args.s3_prefix, args.folder, args.dest_dir)

if __name__ == '__main__':
    main()
