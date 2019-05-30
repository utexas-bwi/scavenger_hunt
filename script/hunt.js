function viewHunt(id) {
    window.location.href = "createhunt.html?type=edit_hunt&id=" + id;
}

function newHunt() {
    window.location.href = "createhunt.html?type=create_hunt";
}

function submitHunt() {
    // extract data from table
    const table = document.getElementById("hunt-table");
    const data = [];
    for (let rdx = 2; rdx < table.rows.length; ++rdx) {
        const rowdata = [];
        const row = table.rows[rdx];
        // skip last two cells but iterate over rest bc last 2 are buttons
        for (let cdx = 0; cdx < row.cells.length - 2; ++cdx) {
            const cell = row.cells[cdx];
            rowdata.push(cell.textContent);
        }
        data.push(rowdata);
    }
    // get hunt ID of hunt
    let id = -1;
    if (window.location.href.indexOf("id") > -1) {
        const vars = {};
        const parts = window.location.href.replace(/[?&]+([^=&]+)=([^&]*)/gi, function(m,key,value) {
            vars[key] = value;
        });
        id = vars["id"];
    }
    // get name of hunt from URL or from page if new hunt
    const new_name = document.getElementById("hunt-name").textContent;
    // send data to php function
    const url = 'http://localhost/script/save_hunt_table.php';
    var hash = firebase.auth().currentUser.uid.hashCode();
    $.ajax({
        type: "POST",
        url,
        data: {
            save_hunt: true,
            hunt_table: data,
            hunt_id:id,
            hunt_name: new_name,
            user_id: hash
        },
        success: function(data) {
            if (!data.error) {
                const message = document.getElementById("save-msg");
                message.classList.remove("hidden-msg");
                message.classList.remove("alert-danger");
                message.classList.add("alert-success");
                message.textContent = "Your hunt has been saved.";
                setTimeout(function() {
                    message.classList.add("hidden-msg");
                    message.classList.remove("alert-success");
                }, 2000);
            } else {
                const message = document.getElementById("save-msg");
                message.classList.remove("hidden-msg");
                message.classList.remove("alert-success");
                message.classList.add("alert-danger");
                message.textContent = data.data;
                setTimeout(function() {
                    message.classList.add("hidden-msg");
                    message.classList.remove("alert-danger");
                }, 10000);
            }
        }
    });    
}
