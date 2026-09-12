import fs from "node:fs/promises";
import { FileBlob, SpreadsheetFile } from "@oai/artifact-tool";

const workbookPath = "C:/Users/slima/Downloads/The Robotic Hand ISU XR LAB/calibration_rebuild/outputs/01a07b6c-8aba-76d3-b7e9-5182967627c8/openmanipulator_fk_ik_validation.xlsx";
const previewDir = "C:/Users/slima/Downloads/The Robotic Hand ISU XR LAB/calibration_rebuild/outputs/01a07b6c-8aba-76d3-b7e9-5182967627c8/verification";
const blob = await FileBlob.load(workbookPath);
const workbook = await SpreadsheetFile.importXlsx(blob);
const summary = await workbook.inspect({ kind: "sheet,table", maxChars: 6000, tableMaxRows: 8, tableMaxCols: 10 });
console.log(summary.ndjson);
const errors = await workbook.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!",
  options: { useRegex: true, maxResults: 100 },
  summary: "final formula error scan",
});
console.log(errors.ndjson);
await fs.mkdir(previewDir, { recursive: true });
for (const name of ["Calibration_Input", "FK_Validation", "IK_FK_Validation", "Calibration_Parameters"]) {
  const preview = await workbook.render({ sheetName: name, autoCrop: "all", scale: 1, format: "png" });
  await fs.writeFile(`${previewDir}/${name}.png`, new Uint8Array(await preview.arrayBuffer()));
  console.log(`rendered ${name}`);
}
