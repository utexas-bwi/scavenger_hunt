var config = {
  apiKey: "AIzaSyCYjHj1mc4vZqxNNYG6NB5S4NWYZ7d07lY",
  authDomain: "scavhunt-login.firebaseapp.com",
  databaseURL: "https://scavhunt-login.firebaseio.com",
  projectId: "scavhunt-login",
  storageBucket: "scavhunt-login.appspot.com",
  messagingSenderId: "446710554532"
};

firebase.initializeApp(config);
firebase.database().enableLogging(true);
console.log("FIREBASE WAS INTIALIZED");
