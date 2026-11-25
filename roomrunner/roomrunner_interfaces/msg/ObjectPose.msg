# One object seen by the overhead camera, in the frame of the array's header (map).
string name          # unique name used in requests and plans, for example "red_block" or "tray"
string kind          # "block" or "tray"
int32 tag_id         # ArUco id of the tag on the object (-1 if not tagged)
geometry_msgs/Pose pose
float32 confidence   # 0 to 1
