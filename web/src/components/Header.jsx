import React from "react";
import { NavLink } from "react-router-dom";

const Header = () => {
  return (
    <header className="bg-themeBlue flex items-center justify-end gap-20 border-b-2 border-white border-b-white border-opacity-90  px-[50px] font-[RobotoMono] text-white">
      <nav className="flex gap-20 py-[30px] text-lg lg:text-2xl">
        <NavLink
          to="/"
          className="hover:text-textWhiteHover active:text-textWhiteActive"
        >
          Map
        </NavLink>
        <NavLink
          to="/route"
          className="hover:text-textWhiteHover active:text-textWhiteActive"
        >
          Route
        </NavLink>
        <NavLink
          to="/control"
          className="hover:text-textWhiteHover active:text-textWhiteActive"
        >
          Control
        </NavLink>
        <NavLink
          to="/info"
          className="hover:text-textWhiteHover active:text-textWhiteActive"
        >
          Info
        </NavLink>
      </nav>
    </header>
  );
};

export default Header;
