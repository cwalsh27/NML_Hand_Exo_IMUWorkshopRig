#from pylsl import resolve_stream
import pylsl

streams = pylsl.resolve_streams()

# Resolve all available LSL streams (waits up to 5 seconds)
print("Looking for LSL streams...")
#streams = resolve_stream()
# Print name and type of each stream
for i, stream in enumerate(streams):
    print(f"{i+1}. Name: {stream.name()}, Type: {stream.type()}, Source ID: {stream.source_id()}")

    inlet = pylsl.StreamInlet(stream)
    print(f"Stream Inlet info: {inlet.info()}")
    print(f"Channel count: {inlet.info().channel_count()}")