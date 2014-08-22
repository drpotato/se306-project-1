# Message from a non-Resident Actor to the Resident, signalling that they are interacting with him.
# He responds by increasing the specified attribute by the specified amount.
# Also contains a boolean indicating whether or not this interaction can be interrupted by another Actor (a lock).
# Alternative to Interaction.
string attribute
int8 amount
bool interruptible