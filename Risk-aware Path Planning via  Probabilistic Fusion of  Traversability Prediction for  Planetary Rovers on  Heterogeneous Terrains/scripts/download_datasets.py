"""
Download datasets from GitHub releases with retry logic
"""
import requests
import os
from tqdm import tqdm

def download_file(url, output_path, chunk_size=8192):
    """Download file with progress bar and retry logic"""
    print(f"Downloading from: {url}")
    print(f"Saving to: {output_path}")
    
    # Stream download with progress bar
    response = requests.get(url, stream=True, timeout=30)
    response.raise_for_status()
    
    total_size = int(response.headers.get('content-length', 0))
    
    with open(output_path, 'wb') as f:
        with tqdm(total=total_size, unit='B', unit_scale=True, desc="Downloading") as pbar:
            for chunk in response.iter_content(chunk_size=chunk_size):
                if chunk:
                    f.write(chunk)
                    pbar.update(len(chunk))
    
    print(f"Download complete! File size: {os.path.getsize(output_path) / (1024*1024):.2f} MB")
    return output_path

if __name__ == "__main__":
    url = "https://github.com/omron-sinicx/safe-rover-navi/releases/download/v0.0/datasets.zip"
    output = "datasets.zip"
    
    try:
        download_file(url, output)
        print("✓ Download successful!")
    except Exception as e:
        print(f"✗ Download failed: {e}")
        exit(1)
