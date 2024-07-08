from __future__ import print_function
import os
import pickle
import sys
from tqdm import tqdm
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.http import MediaIoBaseDownload

# If modifying these SCOPES, delete the file token.pickle.
SCOPES = ['https://www.googleapis.com/auth/drive.readonly']

def authenticate():
    """Authenticate the user and return the Drive service."""
    creds = None
    if os.path.exists('token.pickle'):
        with open('token.pickle', 'rb') as token:
            creds = pickle.load(token)
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(
                './credentials.json', SCOPES)
            creds = flow.run_local_server(port=0)
        with open('token.pickle', 'wb') as token:
            pickle.dump(creds, token)
    return build('drive', 'v3', credentials=creds)

def list_files_in_folder(service, folder_id):
    """List all files in a Google Drive folder."""
    results = service.files().list(
        q=f"'{folder_id}' in parents",
        pageSize=1000,
        fields="nextPageToken, files(id, name, mimeType)"
    ).execute()
    return results.get('files', [])

def download_file(service, file_id, file_name, mime_type, dest_folder):
    """Download a file from Google Drive."""
    file_path = os.path.join(dest_folder, file_name)

    # Check if the file is a Google Docs Editors file
    if mime_type.startswith('application/vnd.google-apps.'):
        export_mime_type = {
            'application/vnd.google-apps.document': 'application/pdf',
            'application/vnd.google-apps.spreadsheet': 'text/csv',
            'application/vnd.google-apps.presentation': 'application/pdf'
        }.get(mime_type, None)

        if export_mime_type:
            request = service.files().export_media(fileId=file_id, mimeType=export_mime_type)
        else:
            print(f"Cannot export file '{file_name}' of type '{mime_type}'")
            return
    else:
        request = service.files().get_media(fileId=file_id)

    with open(file_path, 'wb') as f:
        downloader = MediaIoBaseDownload(f, request)
        done = False
        with tqdm(total=100, desc=f"Downloading {file_name}", unit='%') as pbar:
            while not done:
                status, done = downloader.next_chunk()
                if status:
                    pbar.update(status.progress() * 100)
    print(f"Downloaded {file_name} to {file_path}")

def create_unique_directory(base_dir):
    """Create a unique directory by appending a number if the directory already exists."""
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
        return base_dir

    counter = 1
    new_dir = f"{base_dir}_{counter}"
    while os.path.exists(new_dir):
        counter += 1
        new_dir = f"{base_dir}_{counter}"
    os.makedirs(new_dir)
    return new_dir

if __name__ == '__main__':
    drive_service = authenticate()

    # Default folder ID
    folder_id = "1SGrD8wHhXEra-UZHSY-6gQeqhjScKFGn"

    # Default destination path
    base_dest_path = "./yolov10_data"

    # Create a unique directory
    dest_path = create_unique_directory(base_dest_path)
    print(f"Files will be downloaded to: {dest_path}")

    # List and download files
    files = list_files_in_folder(drive_service, folder_id)
    if not files:
        print(f"No files found in folder with ID {folder_id}.")
    else:
        for file in files:
            download_file(drive_service, file['id'], file['name'], file['mimeType'], dest_path)
        print("\n\n***[COMPLETED !]***")
