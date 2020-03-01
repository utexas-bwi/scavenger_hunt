<?php
include_once 'connect.php';
include_once 'auth.php';

$dbh = connect();
$dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

try {
  $proof_id = $_POST["id"];
  $stmt = $dbh->query("SELECT * FROM proof_table WHERE (proof_id=" . $proof_id . " and verified=1)");
  $stmt->setFetchMode(PDO::FETCH_ASSOC);

  if ($proof = $stmt->fetch())
    echo $proof["correct"];
  else
    echo "-1";

} catch (PDOException $e) {
  die($e->getMessage());
}

?>
