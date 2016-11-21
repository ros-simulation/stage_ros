define qrcode model
(
  color "black"
  size [0.5 0.5 0.0]
  bitmap "bitmaps/fiducial.png"
  boundary 0

  # GUI properties
  gui_nose 0
  gui_grid 0
  gui_outline 0
  gui_move 1

  # determine how the model appears in various sensors
  fiducial_return 1   # Default fiducial ID
  fiducial_key 1      # fiducial_key values of model and fiducial detector must match
  obstacle_return 0
  ranger_return 0
  blob_return 0
  gripper_return 0
)
