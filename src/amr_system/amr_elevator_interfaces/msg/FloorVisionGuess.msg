# Camera floor recognizer's guess for the stop the car just made (published once per door opening).
# floor == "" means "unknown" (no confident answer); the monitor then asks about the ride target.
std_msgs/Header header        # stamp of the newest frame used
string floor                  # floors.yaml floor or pass-floor name, or "" (unknown)
float32 score                 # mean best cosine similarity to that floor's references (0..1)
float32 margin                # score minus the runner-up floor's score
uint32 frames                 # frames used
string model                  # e.g. "facebook/dinov2-small"
string[] reference_floors     # floors that have reference images in the recognizer's gallery
