function addFields(fieldName) {
    // remove all extra form questions specific to a given task
    const addFormTo = document.getElementById("task_form");
    while (addFormTo.childElementCount > 3) {
        addFormTo.removeChild(addFormTo.lastChild);
    }
    // add extra questions based on task chosen
    if (fieldName === "color") {
        const par = document.createElement("p");
        par.textContent = "Pick a color shirt for the robot to find.";
        addFormTo.appendChild(par);
        const select = document.createElement("select");
        select.name = "colors";
        const colors = ["red", "blue", "green", "yellow"];
        for (let i = 0; i < colors.length; ++i) {
            const option = document.createElement("option");
            option.value = colors[i];
            option.textContent = colors[i];
            select.appendChild(option);
        }
        addFormTo.appendChild(select);
        addFormTo.appendChild(document.createElement("br"));
    } else if (fieldName == "target") {
        const par = document.createElement("p");
        par.textContent = "Pick a target object for the robot to find.";
        addFormTo.appendChild(par);
        const select = document.createElement("select");
        select.name = "colors";
        const objects = ["can", "bottle", "pen", "shoe"];
        for (let i = 0; i < objects.length; ++i) {
            const option = document.createElement("option");
            option.value = objects[i];
            option.textContent = objects[i];
            select.appendChild(option);
        }
        addFormTo.appendChild(select);
        addFormTo.appendChild(document.createElement("br"));
    }
    if (!(fieldName === "default")) {
        // add submit button
        const input = document.createElement("input");
        input.type = "submit";
        input.value="Submit Task";
        addFormTo.appendChild(input);
    }
}
