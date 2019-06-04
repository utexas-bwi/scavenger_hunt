<?php
include 'connect.php';

$dbh = connect();

$proof_dir = "../proof";
$hash = hash("sha256", $_FILES["image"]["tmp_name"] . $_FILES["image"]["name"] . $_FILES["image"]["size"]);
$target_path = $proof_dir . "/" . $hash;
$filetype = strtolower(pathinfo($_FILES["image"]["name"], PATHINFO_EXTENSION));

// Check file size
if ($_FILES["image"]["size"] > 500000) {
  echo "[send_proof] Upload failed: file exceeds size limit.\n";
  die();
}

// Allow only certain image formats
if ($filetype != "png" && $filetype != "jpg" && $filetype != "jpeg") {
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
    move_uploaded_file($_FILES["image"]["tmp_name"], $target_path);

    // Log new proof in database
    $query = "insert into proof_table values (0, '" . $user['user_id'] . "', '" . $hash . "', " . $_POST["instr_id"] . ", 0, 0)";
    $stmt = $dbh->query($query);
  } else
    echo "[send_proof] Specified user not found in database!";
} catch (PDOException $e) {
  echo "[send_proof] An error occurred: " . $e->getMessage() . "\n";
}
?>
