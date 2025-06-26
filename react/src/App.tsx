import React, { useMemo, useState } from "react";
import Box from "@mui/material/Box";
import MapPicker from "./map";
import Stack from "@mui/material/Stack";
import TextField from "@mui/material/TextField";
import AppBar from "@mui/material/AppBar";
import Typography from "@mui/material/Typography";
import Toolbar from "@mui/material/Toolbar";
import FormControl from "@mui/material/FormControl";
import InputLabel from "@mui/material/InputLabel";
import Select from "@mui/material/Select";
import MenuItem from "@mui/material/MenuItem";
import Button from "@mui/material/Button";
import * as formik from "formik";
import * as yup from "yup";
import FormHelperText from "@mui/material/FormHelperText";
import { addDoc, collection } from "firebase/firestore";
import { db } from "./Firebase";
import Alert from "@mui/material/Alert";

const DefaultLocation = { lat: 52.01132435503029, lng: 4.3577094736899635 };
const DefaultZoom = 14;
// interfac Values {
//   location: { lat: number; lng: number };
//   initials: string;
//   restaurant: string;
// }
const App = () => {
  const { Formik } = formik;

  const [defaultLocation] = useState(DefaultLocation);
  const [location, setLocation] = useState(defaultLocation);
  const [zoom, setZoom] = useState(DefaultZoom);
  const [isSubmitting, setIsSubmitting] = useState(false);

  function handleChangeLocation(lat: any, lng: any) {
    if (!isSubmitting) {
      setLocation({ lat: lat, lng: lng });
    }
  }

  function handleChangeZoom(newZoom: React.SetStateAction<number>) {
    setZoom(newZoom);
  }

  const initialValues = useMemo(() => {
    return {
      initials: "", //
      restaurants: "", //
      location: { lat: defaultLocation.lat, lng: defaultLocation.lng }, // Initial location
    };
  }, []);
  const validationSchema = yup.object().shape({
    initials: yup
      .string()
      .required("initials are required")
      .max(2, "Max 2 characters")
      .min(2, "2 characters required")
      .matches(/^[A-Za-z]+$/, "Only letters are allowed"),
    restaurants: yup.string().required("restaurant is required"),
  });

  return (
    <Box
      width="100vw"
      height="100vh"
      display="flex"
      flexDirection="column"
      bgcolor={"white"}
    >
      <Box sx={{ flexGrow: 0 }}>
        <AppBar position="static">
          <Toolbar>
            <Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>
              DronebezORgd
            </Typography>
          </Toolbar>
        </AppBar>
      </Box>
      <Box
        sx={{
          flexGrow: 1,
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
          overflow: "auto",
        }}
      >
        <Formik
          onSubmit={(values) => {
            setIsSubmitting(true);
            console.log("Submitted");
            addDoc(collection(db, "requests"), {
              initials: values.initials,
              restaurant: values.restaurants,
              location: location,
            });
          }}
          initialValues={initialValues}
          validationSchema={validationSchema}
        >
          {({
            handleSubmit,
            handleChange,
            handleBlur,
            values,
            touched,
            errors,
            isSubmitting,
          }) => (
            <Stack
              direction="column"
              spacing={2}
              height={"100%"}
              width="100%"
              maxWidth="600px"
              padding="10px"
              boxSizing="border-box"
              style={{ rotate: "0deg" }}
            >
              <MapPicker
                defaultLocation={defaultLocation}
                zoom={zoom}
                style={{ height: "60%" }}
                onChangeLocation={handleChangeLocation}
                onChangeZoom={handleChangeZoom}
                apiKey="AIzaSyAODS4ExHjKCipXO0ZmVaqv_TGPTpWIZ8c"
              />
              <FormControl fullWidth>
                <InputLabel id="demo-simple-select-label">
                  Restaurants
                </InputLabel>
                <Select
                  labelId="demo-simple-select-label"
                  id="restaurants"
                  name="restaurants"
                  label="restaurants"
                  value={values.restaurants}
                  onChange={handleChange}
                  onBlur={handleBlur}
                  error={touched.initials && Boolean(errors.initials)}
                >
                  {restaurants.map((restaurant, index) => (
                    <MenuItem key={index} value={restaurant}>
                      {restaurant}
                    </MenuItem>
                  ))}
                </Select>
                <FormHelperText error>
                  {Boolean(!values.restaurants) ? "restaurant Needed" : ""}
                </FormHelperText>
              </FormControl>
              <TextField
                name="initials"
                id="outlined-basic"
                label="Initials"
                variant="outlined"
                value={values.initials}
                onChange={handleChange}
                onBlur={handleBlur}
                error={touched.initials && Boolean(errors.initials)}
                helperText={touched.initials && errors.initials}
              />
              {
                <Button
                  variant="outlined"
                  onClick={() => handleSubmit()}
                  disabled={isSubmitting}
                  sx={{ position: "relative", overflow: "hidden" }}
                >
                  Submit
                  <span
                    style={{
                      position: "absolute",
                      left: 0,
                      top: "50%",
                      transform: "translateY(-50%)",
                      pointerEvents: "none",
                      animation: isSubmitting
                        ? "plane-fly 1s linear forwards"
                        : "none",
                      transition: "opacity 0.2s",
                      opacity: isSubmitting ? 1 : 0,
                    }}
                  >
                    <svg
                      width="24"
                      height="24"
                      viewBox="0 0 24 24"
                      fill="none"
                      xmlns="http://www.w3.org/2000/svg"
                    >
                      <path
                        d="M2 16L22 12L2 8L2 13L17 12L2 11L2 16Z"
                        fill="#1976d2"
                      />
                    </svg>
                  </span>
                  <style>
                    {`
                    @keyframes plane-fly {
                    0% { left: 0; opacity: 1; }
                    80% { opacity: 1; }
                    100% { left: 100%; opacity: 0; }
                    }
                  `}
                  </style>
                </Button>
              }
              {isSubmitting && (
                <Alert
                  severity="success"
                  variant="filled"
                  sx={{ width: "90%" }}
                >
                  Order submitted
                </Alert>
              )}
            </Stack>
          )}
        </Formik>
      </Box>
    </Box>
  );
};

export default App;
const restaurants = [
  "Pizzeria-Ristorante Fontanella",
  "Domino's",
  "De pizzabakkers",
  "Fratelli",
  "Il Bambino",
  "De Beren",
  "Papa John's",
  "Herdem",
  "Domino's",
  "Porto Bello",
  "Pizza Manhattan",
  "New York Pizza",
  "Einstein",
  "Pavarotti",
  "Pizza Dennis",
  "Domino's",
  "il Peperoncino",
  "Settebello",
  "Klein Italië",
  "Doner15",
  "Settebello",
  "La Bella Pizza",
  "Lasagne Company",
  "Pizzeria da Salvatore",
  "Stromboli",
  "Pizza Hood",
  "San Marco",
  "De Pi",
  "Sapori di Casa",
  "Say Cheese",
];
