import { Navigate, Route, Routes } from "react-router-dom";

import Layout from "./components/Layout";
import AmrControlPage from "./pages/AmrControlPage";
import DashboardPage from "./pages/DashboardPage";
import PackagesPage from "./pages/PackagesPage";

export default function App() {
  return (
    <Routes>
      <Route element={<Layout />}>
        <Route index element={<DashboardPage />} />
        <Route path="packages" element={<PackagesPage />} />
        <Route path="packages/:packageCode" element={<PackagesPage />} />
        <Route path="amr" element={<AmrControlPage />} />
        <Route path="*" element={<Navigate to="/" replace />} />
      </Route>
    </Routes>
  );
}
