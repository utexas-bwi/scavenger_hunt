<?php
function row($data) {
    $id = $data['hunt_id'];
    $name = $data['hunt_name'];
    include '../public_html/components/user-hunt-table-row.html'; 
}

// Create connection
include 'connect.php';
include '../public_html/components/user-hunt-table-header-row.html'; 

$dbh = connect();

try {
    $user_id = $_POST["user_id"];
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    $query = "SELECT hunt_id, hunt_name FROM hunt_table where user_id=" . $user_id;
    $stmt = $dbh->query($query);
    $stmt->setFetchMode(PDO::FETCH_ASSOC);
    while ($data = $stmt->fetch()):
        row($data);
    endwhile;
    $stmt = null; 
} catch (PDOException $e) {
    die($e->getMessage());
}

// Close connection
$dbh = null;
?>
