<?php
include_once 'connect.php';
include_once 'auth.php';

$dbh = connect();
$dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

try {
  $proof_id = $_POST["id"];
  $stmt = $dbh->query("SELECT * FROM proof_table WHERE proof_id=" . $proof_id);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);

  if ($proof = $stmt->fetch())
    echo $proof["filename"];
  else
    die("FATAL: NO FILENAME ON RECORD FOR PROOF " . $proof_id);

} catch (PDOException $e)
  die($e->getMessage());
}

?>
