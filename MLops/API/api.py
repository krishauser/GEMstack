import requests
import os
from tqdm import tqdm
from urllib.parse import unquote, urljoin
import time
import zipfile

class APIClient:
    def __init__(self, base_url):
        self.base_url = base_url
        self.save_directory = os.getcwd()

    def get_file(self, file_id):
        # response = requests.get(f"{self.base_url}/files/{file_id}", stream=True)
        response = requests.get(file_id)
        if response.status_code == 200:
            total_size_in_bytes = int(response.headers.get('content-length', 0))
            progress_bar = tqdm(total=total_size_in_bytes, unit='iB', unit_scale=True)
            content_disposition = response.headers.get('content-disposition')
            if content_disposition:
                filename = unquote(content_disposition.split('filename=')[-1].strip('"'))
                save_path = f"{self.save_directory}/{filename}"
            else:
                save_path = f"{self.save_directory}/default_filename.ext"
                print("Filename not provided in the response headers, saving as default_filename.ext")
            
            with open(save_path, 'wb') as f:
                for chunk in response.iter_content(chunk_size=1024):
                    progress_bar.update(len(chunk))
                    if chunk:
                        f.write(chunk)
            progress_bar.close()
            print(f"File downloaded successfully and saved to {save_path}")
        else:
            print(f"Failed to download the file. Status code: {response.status_code}")

    def upload_file(self, file_path):
        if not os.path.isfile(file_path):
            print("File does not exist.")
            return
        
        try:
            with open(file_path, 'rb') as f:
                files = {'file': f}
                response = requests.post(f"{self.base_url}/files", files=files)
                if response.status_code == 200:
                    return response.json()
                else:
                    response.raise_for_status()
        except requests.RequestException as e:
            print(f"Request failed: {e}")

    # def convert_rosbag(self, file_path):
    #     if not os.path.isfile(file_path):
    #         print("File does not exist.")
    #         return
        
    #     try:
    #         with open(file_path, 'rb') as f:
    #             convert_url = urljoin(self.base_url, '/convert_rosbag')
    #             files = {'file': (os.path.basename(file_path), f, 'application/octet-stream')}
    #             response = requests.post(convert_url, files=files)
                
    #             if response.status_code == 200:
    #                 print("File uploaded successfully, conversion started.")
    #                 conversion_result = response.json()
    #                 if 'result_url' in conversion_result:
    #                     self.download_conversion_result(conversion_result['result_url'])
    #                 elif 'job_id' in conversion_result:
    #                     self.poll_and_download_result(conversion_result['job_id'])
    #                 else:
    #                     print("Conversion initiated, but no result URL or job ID was provided.")
    #             else:
    #                 print(f"Failed to upload the file. Status code: {response.status_code}")
    #                 response.raise_for_status()
    #     except requests.RequestException as e:
    #         print(f"Request failed: {e}")

    # def poll_and_download_result(self, job_id):
    #     result_url = urljoin(self.base_url, f'/conversion_result/{job_id}')
    #     while True:
    #         response = requests.get(result_url)
    #         if response.status_code == 200:
    #             result = response.json()
    #             if result.get('status') == 'completed':
    #                 self.download_conversion_result(result['download_url'])
    #                 break
    #             elif result.get('status') == 'failed':
    #                 print("Conversion failed.")
    #                 break
    #         else:
    #             print("Error polling the conversion result.")
    #         time.sleep(10) 

    # def download_conversion_result(self, result_url):
    #     response = requests.get(result_url, stream=True)
    #     if response.status_code == 200:
    #         zip_path = os.path.join(self.save_directory, 'conversion_result.zip')
    #         with open(zip_path, 'wb') as f:
    #             for chunk in response.iter_content(chunk_size=128):
    #                 f.write(chunk)
    #         print("Conversion result downloaded. Extracting...")
    #         with zipfile.ZipFile(zip_path, 'r') as zip_ref:
    #             zip_ref.extractall(self.save_directory)
    #         print("Result extracted successfully.")
    #     else:
    #         print(f"Failed to download the conversion result. Status code: {response.status_code}")
                
