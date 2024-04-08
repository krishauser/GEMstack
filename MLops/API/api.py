import requests
import os
from tqdm import tqdm
from urllib.parse import unquote, urljoin

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
    #                 return response.json()
    #             else:
    #                 print(f"Failed to upload the file. Status code: {response.status_code}")
    #                 response.raise_for_status()
    #     except requests.RequestException as e:
    #         print(f"Request failed: {e}")
                
