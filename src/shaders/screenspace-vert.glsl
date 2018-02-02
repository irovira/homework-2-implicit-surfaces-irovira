#version 300 es

precision highp float;

in vec4 vs_Pos;

out vec2 fs_Pos; //this is used for the fragCoord in FS

void main() {
	// TODO: Pass relevant info to fragment
	gl_Position = vs_Pos;
	fs_Pos = vs_Pos.xy;
}
