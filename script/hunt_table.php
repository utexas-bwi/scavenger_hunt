<?php
function row($data) {
    $task = $data['task_type'];
    $param = $data['param_value'];
    include '../public_html/components/hunt-table-row.html'; 
}

// Create connection
include 'connect.php';
include 'auth.php';
if (isset($_GET['id'])) {
    $id = $_GET['id'];
    $dbh = connect();
    try {
        $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
        $query = "SELECT task_type, param_value FROM hunt_instructions_table where hunt_id=" . $id;
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
}
?>
