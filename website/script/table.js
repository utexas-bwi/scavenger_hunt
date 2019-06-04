/* 
 * toggle editabiity of cell: makes editable if not editable and vice versa
 *
 * parameters:
 *      span: the span containing the pencil glyphicon that was clicked
 *          expects the nesting to be
 *          <tr>
 *              <td>
 *                  <span> pencil icon </span>
 *              </td>
 *          </tr>
 *      start: the cell to start at when making editable
 *          1 = first child, 3 = second, 5 = third, etc.
 *          start at 1 if you want whole row to be editable
 *
 * preconditions:
 *      - span follows proper nesting (noted above)
 *      - start is a number > 1 and < number of child nodes
 *      - tr is formatted to have two uneditable child nodes at end
 *          (in our case, the pencil and x icons)
 * */

function editable(span, start) {
    const tr = span.parentNode.parentNode;
    if (span.classList.contains("glyphicon-pencil")) {
        for (let i = start; i < tr.childNodes.length - 4; i = i + 2) {
            tr.childNodes[i].setAttribute("contentEditable","true");
            tr.childNodes[i].classList.add("editable");
        }
        span.classList.remove("glyphicon-pencil");
        span.classList.add("glyphicon-ok");
    } else {
        for (let i = start; i < tr.childNodes.length - 4; i = i + 2) {
            tr.childNodes[i].setAttribute("contentEditable","false");
            tr.childNodes[i].classList.remove("editable");
        }
        span.classList.add("glyphicon-pencil");
        span.classList.remove("glyphicon-ok");
    }
}

/*
 * add row to task table
 *
 * parameters:
 *      cell: the span component with the "add" button, expected nesting:
 *          <table> <-- table to add the row to
 *              <tr>
 *                  <th>
 *                      <span /> <-- span with the add button
 *                  </th>
 *              </tr>
 *          </table>
 * */

function addRow(cell) {
    const table = cell.parentNode.parentNode.parentNode;
    const row = document.getElementById("tr-hide").cloneNode(true);
    row.classList.remove("hide");
    row.id = "";
    row.classList.remove("table-line");
    // get last used ID and add next ID to row
    let nextId = 1;
    if (table.childNodes.length > 6) {
        let sub = 1;
        let check = table.childNodes[table.childNodes.length - sub];
        while (check.childNodes.length <= 1 || !check.childNodes[1].textContent) {
            ++sub;
            check = table.childNodes[table.childNodes.length - sub];
        }
        nextId = parseInt(check.childNodes[1].textContent) + 1;
    }
    row.childNodes[1].textContent = nextId; 
    editable(row.childNodes[13].childNodes[1], 1);
    table.appendChild(row);
}

/*
 * remove a row from the table given a cell in that row
 *
 * parameters:
 *      cell: the span component with the "remove" button
 *      expected nesting is the same as the addRow() method
 */
function removeRow(cell) {
    const table = cell.parentNode.parentNode.parentNode;
    table.deleteRow(cell.parentNode.parentNode.rowIndex);
}

function saveChanges() {
    // extract data from table
    const table = document.getElementById("task-table");
    const data = [];
    // iterate over rows except first two rows (headers and hidden empty row)
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
    // send data to php function
    const url = 'http://localhost/script/save_task_table.php';
    $.ajax({
        type: "POST",
        url,
        data: {save_tasks: true, task_table: data},
        success: function(data){
            console.log(data);
            const message = document.getElementById("save-msg");
	    message.classList.remove("hidden-msg");
	    message.classList.add("alert-success");
	    message.textContent = "The task library has been updated.";
	    setTimeout(function(){
		message.classList.add("hidden-msg");
		message.classList.remove("alert-success");
	    }, 2000);
	},
	failure: function(data) {
	    console.log(data);
	    const message = document.getElementById("save-msg");
	    message.classList.remove("hidden-msg");
	    message.classList.add("alert-danger");
	    message.textContent = "The task library failed to update.";
	    setTimeout(function(){
		message.classList.add("hidden-msg");
		message.classList.remove("alert-danger");
	    }, 5000);
	}
    });    
}

/*
 * add a task to the hunt table
 * parameters:
 *      task: text to add
 */

function addTask(task) {
    const table = document.getElementById("hunt-table");
    const row = document.getElementById("tr-hide").cloneNode(true);
    row.childNodes[1].textContent = task;
    row.classList.remove("hide");
    row.classList.remove("table-line");
    editable(row.childNodes[5].childNodes[1], 3);
    table.appendChild(row);
}
