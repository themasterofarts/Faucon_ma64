
// Hôte qui sert la page : en local "localhost", depuis une tablette l'IP du robot
// (ihm_lan:=true). rosbridge et la caméra tournent sur cette même machine.
export const ROBOT_HOST = (typeof window !== "undefined" && window.location.hostname) || "localhost";
