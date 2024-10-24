#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "common.h"

namespace py = pybind11;

void physicsStepWrapper(ObjectState &ball, const py::list playersList, const float duration, const py::array_t<uint8_t> &scoresArray)
{
    std::vector<Player> players = playersList.cast<std::vector<Player>>();
    physicsStep(ball, players, true, duration, (uint8_t *)scoresArray.data());
    for (size_t i = 0; i < players.size(); ++i)
    {
        playersList[i] = players[i];
    }
}

PYBIND11_MODULE(hacker_league, m)
{
    py::class_<ObjectState>(m, "State")
        .def(py::init<>())
        .def_readwrite("position", &ObjectState::position)
        .def_readwrite("velocity", &ObjectState::velocity)
        .def_readwrite("orientation", &ObjectState::orientation);
    py::class_<Action>(m, "Action")
        .def(py::init<>())
        .def_readwrite("throttle", &Action::throttle)
        .def_readwrite("steering", &Action::steering);
    py::class_<Player>(m, "Agent")
        .def(py::init<>())
        .def_readwrite("state", &Player::state)
        .def_readwrite("action", &Player::action);
    m.def("step", &physicsStepWrapper, py::arg("ball"), py::arg("agents"), py::arg("duration"), py::arg("scores"));
}
