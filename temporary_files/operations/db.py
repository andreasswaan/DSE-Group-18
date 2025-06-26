import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

# Use a service account.
cred = credentials.Certificate(
    "temporary_files\operations\dronebezorgd-7c265-firebase-adminsdk-fbsvc-3b56c8e485.json"
)

app = firebase_admin.initialize_app(cred)

db = firestore.client()
