"""
Robust Dataset Downloader with Retry and Resume
Handles unstable connections with automatic retry and resume from partial downloads
"""
import requests
import os
import time
from pathlib import Path

def download_with_resume(url: str, output_path: str, chunk_size: int = 8192, max_retries: int = 10):
    """
    Download file with resume capability and retry logic
    
    Args:
        url: Download URL
        output_path: Output file path
        chunk_size: Download chunk size in bytes
        max_retries: Maximum number of retry attempts
    """
    output_file = Path(output_path)
    temp_file = Path(f"{output_path}.part")
    
    # Check if partial download exists
    if temp_file.exists():
        downloaded_size = temp_file.stat().st_size
        print(f"📂 Found partial download: {downloaded_size / (1024*1024):.2f} MB")
    else:
        downloaded_size = 0
    
    # Get total file size
    headers = {}
    if downloaded_size > 0:
        headers['Range'] = f'bytes={downloaded_size}-'
    
    retry_count = 0
    
    while retry_count < max_retries:
        try:
            print(f"\n{'='*70}")
            print(f"Download Attempt {retry_count + 1}/{max_retries}")
            print(f"{'='*70}")
            print(f"URL: {url}")
            print(f"Output: {output_path}")
            if downloaded_size > 0:
                print(f"Resuming from: {downloaded_size / (1024*1024):.2f} MB")
            
            # Make request with timeout
            response = requests.get(url, headers=headers, stream=True, timeout=30)
            response.raise_for_status()
            
            # Get total size
            if 'Content-Length' in response.headers:
                total_size = int(response.headers['Content-Length'])
                if downloaded_size > 0:
                    total_size += downloaded_size
            else:
                total_size = None
            
            print(f"Total size: {total_size / (1024*1024):.2f} MB" if total_size else "Unknown size")
            
            # Download with progress
            mode = 'ab' if downloaded_size > 0 else 'wb'
            
            with open(temp_file, mode) as f:
                start_time = time.time()
                last_update = start_time
                
                for chunk in response.iter_content(chunk_size=chunk_size):
                    if chunk:
                        f.write(chunk)
                        downloaded_size += len(chunk)
                        
                        # Update progress every 2 seconds
                        current_time = time.time()
                        if current_time - last_update >= 2:
                            elapsed = current_time - start_time
                            speed = downloaded_size / elapsed if elapsed > 0 else 0
                            
                            if total_size:
                                progress = (downloaded_size / total_size) * 100
                                eta = (total_size - downloaded_size) / speed if speed > 0 else 0
                                print(f"\r⬇ {progress:.1f}% | "
                                      f"{downloaded_size/(1024*1024):.1f}/{total_size/(1024*1024):.1f} MB | "
                                      f"{speed/(1024):.1f} KB/s | "
                                      f"ETA: {eta/60:.0f}m", end='', flush=True)
                            else:
                                print(f"\r⬇ {downloaded_size/(1024*1024):.1f} MB | "
                                      f"{speed/(1024):.1f} KB/s", end='', flush=True)
                            
                            last_update = current_time
            
            # Download complete
            print(f"\n\n✓ Download complete!")
            
            # Rename temp file to final file
            if output_file.exists():
                output_file.unlink()
            temp_file.rename(output_file)
            
            print(f"✓ Saved to: {output_file}")
            print(f"✓ Size: {output_file.stat().st_size / (1024*1024):.2f} MB")
            
            return True
            
        except (requests.exceptions.RequestException, IOError) as e:
            retry_count += 1
            print(f"\n\n✗ Download failed: {e}")
            
            if retry_count < max_retries:
                wait_time = min(2 ** retry_count, 60)  # Exponential backoff, max 60s
                print(f"⏳ Retrying in {wait_time} seconds...")
                time.sleep(wait_time)
            else:
                print(f"\n✗ Max retries ({max_retries}) reached. Download failed.")
                return False
    
    return False


def main():
    """Main download function"""
    url = "https://github.com/omron-sinicx/safe-rover-navi/releases/download/v0.0/datasets.zip"
    output = "datasets.zip"
    
    print("="*70)
    print("ROBUST DATASET DOWNLOADER")
    print("="*70)
    print("Features:")
    print("  ✓ Automatic resume from partial downloads")
    print("  ✓ Retry with exponential backoff")
    print("  ✓ Progress tracking")
    print("  ✓ Connection timeout handling")
    print("="*70)
    
    success = download_with_resume(url, output, max_retries=20)
    
    if success:
        print("\n" + "="*70)
        print("✓ DOWNLOAD SUCCESSFUL!")
        print("="*70)
        print("\nNext steps:")
        print("1. Extract: Expand-Archive -Path datasets.zip -DestinationPath .\\ -Force")
        print("2. Run: python scripts/run_pipeline_optimized.py")
    else:
        print("\n" + "="*70)
        print("✗ DOWNLOAD FAILED")
        print("="*70)
        print("\nAlternative options:")
        print("1. Download manually from:")
        print("   https://github.com/omron-sinicx/safe-rover-navi/releases/tag/v0.0")
        print("2. Use a download manager (e.g., IDM, wget)")
        print("3. Try again later when connection is more stable")


if __name__ == "__main__":
    main()
