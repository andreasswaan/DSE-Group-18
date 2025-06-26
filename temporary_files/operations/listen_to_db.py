# Create an Event for notifying main thread.
import threading

from temporary_files.operations.db import db


callback_done = threading.Event()


# Create a callback on_snapshot function to capture changes
def on_snapshot(col_snapshot, changes, read_time):
    for doc in col_snapshot:
        print(f"{doc.to_dict()["loglat"].latitude}")
    callback_done.set()


col_query = db.collection("requests")

# Watch the collection query
query_watch = col_query.on_snapshot(on_snapshot)

while True:
    pass
