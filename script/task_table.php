<?php
function row($data) {
    $task = $data['task_type'];
    $param = $data['param_name'];
    $proof = $data['proof_type'];
    $score = $data['score'];
    $descr = $data['description'];
    include 'components/task-table-row.html';
}

/*row('Color Shirt', 'color', 'jpg', '100', 'Take a picture of a person who wears a specific shirt');
row('Target Search', 'object', 'jpg', '100', 'Take a picture of the specified object');
row('Human Following', '', 'jpg', '100', 'Follow a human');

row('Object Delivery', '', 'jpg', '150', 'Find and deliver an object.');
 */
$username = "bwi";
$password = "segbot3768";


// Create connection
//$con = mysqli_connect("localhost","bwi","segbot3768");
$dbh = new PDO('mysql:host = localhost:3306; dbname=scavenger_hunt', $username, $password);

try {
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    $stmt = $dbh->query("SELECT * FROM task_table");
    $stmt->setFetchMode(PDO::FETCH_ASSOC);
    while ($data = $stmt->fetch()):
        row($data);
    endwhile; 
} catch (PDOException $e) {
        die($e->getMessage());
}

// Close connection
?>
