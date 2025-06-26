# Create an Event for notifying main thread.
import threading
from temporary_files.operations.db import db

callback_done = threading.Event()


# Create a callback on_snapshot function to capture changes
orders = []


def on_snapshot(col_snapshot, changes, read_time):
    print(f"Received {len(changes)} documents")
    print(changes)
    for change in changes:
        print(f"{change.document.to_dict()}")
        order = change.document.to_dict()
        order["id"] = change.document.id  # Add document ID to the order
        orders.append(order)
    callback_done.set()
    print(orders)


col_query = db.collection("requests")

# Watch the collection query
query_watch = col_query.on_snapshot(on_snapshot)

while True:
    pass
