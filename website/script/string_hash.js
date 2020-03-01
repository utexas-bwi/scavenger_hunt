/**
 * Overrides String type's hash function with one identical to Java's 32-bit
 * string hash algorithm.
 * 
 * This is primarily used to convert Firebase's string user UID into an integer
 * ID for insertion into SQL tables. This is only okay to do because 1) Firebase
 * user UIDs are randomly generated and pose no threat of SQL injection (I don't
 * think) and 2) hash collisions are very unlikely because the site won't have
 * nearly 2^32 users.
 */
String.prototype.hashCode = function() {
  var hash = 0, i, chr;
  if (this.length === 0)
    return hash;
  for (i = 0; i < this.length; i++) {
    chr = this.charCodeAt(i);
    hash = ((hash << 5) - hash) + chr;
    hash |= 0;
  }
  return hash;
};