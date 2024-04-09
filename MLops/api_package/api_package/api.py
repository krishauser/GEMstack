import requests
import os
from tqdm import tqdm
from pprint import pprint
from urllib.parse import unquote, urljoin

class APIClient:
    def __init__(self, base_url):
        self.base_url = base_url
        self.save_directory = os.getcwd()
        
    def list_models_info(self):
        response = requests.get(f"{self.base_url}/models")
        if response.status_code == 200:
            list_model = response.json()
            print("List of models avaliable on server:\n")
            for info in list_model:
                pprint(info)
                print("\n")
            return list_model
        else:
            print(f"Failed to list models. Status code: {response.status_code}")
    
    def model_info(self, model_id):
        response = requests.get(f"{self.base_url}/models/{model_id}")
        if response.status_code == 200:
            model_info = response.json()
            pprint(model_info)
            return model_info
        elif response.status_code == 404:
            print(f"Error: {response.json()['error']}")
        else:
            print(f"Failed to get model info. Status code: {response.status_code}")
            
    def model_update_description(self, model_id, new_description):
        response = requests.put(f"{self.base_url}/models/{model_id}", json={"Description": new_description})
        if response.status_code == 200:
            print("Model description updated successfully")
        elif response.status_code == 400:
            print(f"Error: {response.json()['error']}")
        else:
            print(f"Failed to update model description. Status code: {response.status_code}")

    def model_download(self, model_id):
        response = requests.get(f"{self.base_url}/models/retrieval/{model_id}", stream=True)
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
        elif response.status_code == 404:
            print(f"Error: {response.json()['error']}")
        else:
            print(f"Failed to download the file. Status code: {response.status_code}")

    def model_upload(self, file_path):
        try:
            with open(file_path, 'rb') as f:
                files = {'file': f}
                response = requests.post(f"{self.base_url}//models/upload", files=files)
                if response.status_code == 200:
                    print(f"Message: {response.json()['message']}. Model name: {response.json()['filename']}")
                elif response.status_code == 400:
                    print(f"Error: {response.json()['error']}")
                else:
                    print(f"Failed to upload the file. Status code: {response.status_code}")
        except FileNotFoundError:
            print(f"File not found: {file_path}")
        except PermissionError:
            print(f"Permission denied: {file_path}")
        except Exception as e:
            print(f"An error occurred: {e}")
            
    def list_datasets_info(self):
        response = requests.get(f"{self.base_url}/datasets")
        if response.status_code == 200:
            list_model = response.json()
            print("List of datasets avaliable on server:\n")
            for info in list_model:
                pprint(info)
                print("\n")
            return list_model
        else:
            print(f"Failed to list datasets. Status code: {response.status_code}")
    
    def dataset_info(self, dataset_id):
        response = requests.get(f"{self.base_url}/datasets/{dataset_id}")
        if response.status_code == 200:
            dataset_info = response.json()
            pprint(dataset_info)
            return dataset_info
        elif response.status_code == 404:
            print(f"Error: {response.json()['error']}")
        else:
            print(f"Failed to get dataset info. Status code: {response.status_code}")
            
    def dataset_update_description(self, dataset_id, new_description):
        response = requests.put(f"{self.base_url}/datasets/{dataset_id}", json={"Description": new_description})
        if response.status_code == 200:
            print("Dataset description updated successfully")
        elif response.status_code == 400:
            print(f"Error: {response.json()['error']}")
        else:
            print(f"Failed to update dataset description. Status code: {response.status_code}")

    def dataset_download(self, dataset_id):
        response = requests.get(f"{self.base_url}/datasets/retrieval/{dataset_id}", stream=True)
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
        elif response.status_code == 404:
            print(f"Error: {response.json()['error']}")
        else:
            print(f"Failed to download the file. Status code: {response.status_code}")

    def dataset_upload(self, file_path):
        try:
            with open(file_path, 'rb') as f:
                files = {'file': f}
                response = requests.post(f"{self.base_url}/datasets/upload", files=files)
                if response.status_code == 200:
                    print(f"Message: {response.json()['message']}. Model name: {response.json()['filename']}")
                elif response.status_code == 400:
                    print(f"Error: {response.json()['error']}")
                else:
                    print(f"Failed to upload the file. Status code: {response.status_code}")
        except FileNotFoundError:
            print(f"File not found: {file_path}")
        except PermissionError:
            print(f"Permission denied: {file_path}")
        except Exception as e:
            print(f"An error occurred: {e}")
            
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
 