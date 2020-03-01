<?php
include 'connect.php';

$dbh = connect();

date_default_timezone_set("America/Chicago");

$proof_dir = "../proof/";
$hash = hash("sha256", $_FILES["image"]["tmp_name"] . $_FILES["image"]["name"] . $_FILES["image"]["size"]);
$filetype = strtolower(pathinfo($_FILES["image"]["name"], PATHINFO_EXTENSION));
$target_filename = date('Y-m-d') . date("H:i:s") . "-" . $hash . "." . $filetype;
$telem_tag = "[" . $_POST["email"] . "//send_proof" . "]";

// Check if the hunt in question is ongoing
$ongoing = false;

try {
  $stmt = $dbh->query("select * from hunt_instructions_table where hunt_instr_id=" . $_POST["instr_id"]);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);
  $reference_instr = $stmt->fetch();

  if ($reference_instr) {
    $stmt = $dbh->query("select * from hunt_table where hunt_id=" . $reference_instr["hunt_id"]);
    $reference_hunt = $stmt->fetch();

    if ($reference_hunt) {
      $current_date = date('Y-m-d');
      $release_date = date('Y-m-d', strtotime($reference_hunt["release_date"]));

      error_log($reference_hunt["end_date"]);

      if ($reference_hunt["end_date"] == "")
        $end_date = NULL;
      else
        $end_date = date('Y-m-d', strtotime($reference_hunt["end_date"]));

      if (is_null($end_date) || $current_date >= $release_date && $current_date <= $end_date)
        $ongoing = true;
    }
  }

} catch (PDOException $e) {
  echo $telem_tag . " Upload failed: " . $e->getMessage() . "\n";
  die();
}

if (!$ongoing) {
  echo $telem_tag . " Upload failed: the specified hunt is not currently active!\n";
  die();
}

// Check file size
if ($_FILES["image"]["size"] > 500000) {
  echo $telem_tag . " Upload failed: file exceeds size limit.\n";
  die();
}

$allowedFiletypes = array("png", "jpg", "jpeg", "mp4");
// Allow only certain image formats
if (!in_array($filetype, $allowedFiletypes)) {
  echo $telem_tag . " Upload failed: file extension not supported.\n";
  die();
}

try {
  // Check user exists
  $query = "select * from user_table where (email='" . $_POST["email"] . "' and pass_hash=" . $_POST["pass_hash"] . ")";
  $stmt = $dbh->query($query);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);
  $user = $stmt->fetch();
  $user_exists = !!$user;

  if ($user_exists) {
    // Move tmp to the proof directory
    move_uploaded_file($_FILES["image"]["tmp_name"], $proof_dir . $target_filename);

    // Log new proof in database
    $query = "insert into proof_table values (0, '" . $user['user_id'] . "', '" . $target_filename . "', " . $_POST["instr_id"] . ", " . $_POST["time"] . ", 0, 0)";
    $stmt = $dbh->query($query);

    // Get the UID that was generated for that proof
    $query = "select * from proof_table where filename='" . $target_filename . "'";
    $stmt = $dbh->query($query);
    $proof = $stmt->fetch();
    $proof_id = $proof["proof_id"];

    echo $telem_tag . " Upload successful. Proof ID: " . $proof_id . "\n";
  } else
    echo $telem_tag . " Upload failed: specified user not found in database. Are your credentials correct?\n";
} catch (PDOException $e) {
  echo $telem_tag . " Upload failed: " . $e->getMessage() . "\n";
  die();
}
?>
