<?php
include 'connect.php';

$dbh = connect();

date_default_timezone_set("America/Chicago");

$proof_dir = "../proof/";
$hash = hash("sha256", $_FILES["image"]["tmp_name"] . $_FILES["image"]["name"] . $_FILES["image"]["size"]);
$filetype = strtolower(pathinfo($_FILES["image"]["name"], PATHINFO_EXTENSION));
$target_filename = date('y-m-d') . date("H:i:s") . "-" . $hash . "." . $filetype;

// Check if the hunt in question is ongoing
$ongoing = true;

try {
  $stmt = $dbh->query("select * from hunt_instructions_table where hunt_instr_id=" . $_POST["instr_id"]);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);
  $reference_instr = $stmt->fetch();

  if ($reference_instr) {
    $stmt = $dbh->query("select * from hunt_table where hunt_id=" . $reference_instr["hunt_id"]);
    $reference_hunt = $stmt->fetch();

    if ($reference_hunt) {
      $release_date = $reference_hunt["release_date"];
      $end_date = $reference_hunt["end_date"];
      $current_date = date('y-m-d');
      if ($current_date >= $release_date && $current_date <= $end_date)
        $ongoing = true;
    }
  }

} catch (PDOException $e) {
  echo "[send_proof] An error occurred: " . $e->getMessage() . "\n";
  die();
}

if (!$ongoing) {
  echo("[send_proof] The specified hunt has expired!");
  die();
}

// Check file size
if ($_FILES["image"]["size"] > 500000) {
  echo "[send_proof] Upload failed: file exceeds size limit.\n";
  die();
}

$allowedFiletypes = array("png", "jpg", "jpeg", "mp4");
// Allow only certain image formats
if (!in_array($filetype, $allowedFiletypes)) {
  echo "[send_proof] Upload failed: file extension not supported.\n";
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
  } else
    echo "[send_proof] Specified user not found in database!\n";
} catch (PDOException $e) {
  echo "[send_proof] An error occurred: " . $e->getMessage() . "\n";
  die();
}
?>
